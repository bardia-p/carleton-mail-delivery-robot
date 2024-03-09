$(document).ready( function() {
    /**
     * The JavaScript AJAX call for when a delivery status is updated (added to the textbox).
     */
        const textbox = document.getElementById("logs");
        const splitRef = window.location.href.split("/");
        const id = splitRef[splitRef.length - 1];
        console.log(`id from param ${id}`);
        console.log(id);
        $.ajax({
            type: "GET",
            url: `/api/v1/getDeliveryStatus/${id}`,
            dataType: "json",
            contentType: "application/json",
            success: function (res) {
                console.log('Delivery status gotten');
                console.log(res);
                if (res) {
                    if (res["status"] !== res["status"].prev) {
                        textbox.append(res["status"]);
                    }
                }
            },
            error: function (xhr, status, error) {
                // error handling
                console.error('Error updating status:', error);
                setToast("error", "Something went wrong", "Could not create user");
            }
        });
})