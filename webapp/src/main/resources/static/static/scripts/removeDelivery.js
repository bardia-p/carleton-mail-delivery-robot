const robotInfoButton = $("#removeDelivery-submit");


/**
 * The JavaScript AJAX call for when robot information is retrieved.
 */
robotInfoButton.click((e) => {
    e.preventDefault();
    const splitRef = window.location.href.split("/");
    const id1 = splitRef[splitRef.length - 2];
    const id2 = splitRef[splitRef.length - 1];
    $.ajax({
        type: "POST",
        url: `/api/v1/removeDelivery/${id1}/${id2}`,
        success: function (res) {
            console.log('User registered successfully');
            if (res === 200) {
                setToast("success", "", "Successfully registered user", true);
                window.location.href = '/';
            }
            else if (res === 401) errorMessage.text("This username already exists!")
        },
        error: function (xhr, status, error) {
            // error handling
            console.error('Error creating user:', error);
            setToast("error", "Something went wrong", "Could not create user");
        }
    });
});