$(document).ready( function() {
    /**
     * The JavaScript AJAX call for when a delivery status is updated (added to the textbox).
     */
    const textbox = document.getElementById("logs");
    const splitRef = window.location.href.split("/");
    const id = splitRef[splitRef.length - 1];
    console.log(`id from param ${id}`);
    console.log(id);
    const updateStatus = () => {
        $.ajax({
            type: "GET",
            url: `/api/v1/getDeliveryStatus/${id}`,
            dataType: "json",
            contentType: "application/json",
            success: function (res) {
                console.log('Delivery status gotten');
                console.log(res);
                if (res) {
                    let textBoxValue = "";
                    for(let q of Object.keys(res["statuses"])) {
                        console.log(q);
                        textBoxValue += res["statuses"][q] + "\n";
                    }
                    textbox.value = textBoxValue;
                }},
            error: function (xhr, status, error) {
                // error handling
                console.error('Error updating status:', error);
                setToast("error", "Something went wrong", "Could not create user");
            }});
        };

    setInterval(() => {updateStatus()}, 2000);
})